import { ComponentFixture, TestBed } from '@angular/core/testing';

import { OverviewMainIdeaComponent } from './overview-main-idea.component';

describe('OverviewMainIdeaComponent', () => {
  let component: OverviewMainIdeaComponent;
  let fixture: ComponentFixture<OverviewMainIdeaComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ OverviewMainIdeaComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(OverviewMainIdeaComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
